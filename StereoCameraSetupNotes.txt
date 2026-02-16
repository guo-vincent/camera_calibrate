import {
  Box,
  Button,
  CircularProgress,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  FormControl,
  Grid,
  InputLabel,
  MenuItem,
  Select,
  SelectChangeEvent,
  TextField,
  Typography,
} from "@mui/material";
import React, { useEffect, useState } from "react";

type Opportunity = {
  id: string | number;
  name: string;
};

type LogHoursModalProps = {
  isOpen: boolean;
  onClose: () => void;
  opportunities: Opportunity[];
  volunteerId: string | number;
  onSuccess?: () => void;
};

const todayIso = (): string => new Date().toISOString().slice(0, 10);

const LogHoursModal: React.FC<LogHoursModalProps> = ({
  isOpen,
  onClose,
  opportunities,
  volunteerId,
  onSuccess,
}) => {
  const [opportunityId, setOpportunityId] = useState<string>("");
  const [date, setDate] = useState<string>(todayIso());
  const [hours, setHours] = useState<string>("0");
  const [notes, setNotes] = useState<string>("");
  const [submitting, setSubmitting] = useState(false);

  useEffect(() => {
    if (isOpen && opportunities.length > 0 && !opportunityId) {
      setOpportunityId(String(opportunities[0].id));
    }
  }, [isOpen, opportunities, opportunityId]);

  // Reset local state helper
  const reset = (): void => {
    setOpportunityId("");
    setDate(todayIso());
    setHours("0");
    setNotes("");
    setSubmitting(false);
  };

  const handleClose = (): void => {
    reset();
    onClose();
  };

  const handleSelectChange = (e: SelectChangeEvent<string>): void => {
    setOpportunityId(e.target.value);
  };

  const handleSubmit = async (
    e: React.FormEvent<HTMLFormElement>,
  ): Promise<void> => {
    e.preventDefault();

    // Basic client validation
    if (!opportunityId) {
      alert("Please select an opportunity.");
      return;
    }
    if (!date || Number.isNaN(Date.parse(date))) {
      alert("Please enter a valid date.");
      return;
    }
    const hoursNum = Number.parseFloat(hours);
    if (Number.isNaN(hoursNum) || hoursNum <= 0) {
      alert("Please enter hours greater than 0.");
      return;
    }

    setSubmitting(true);
    try {
      const payload = {
        opportunityId: String(opportunityId),
        date,
        hours: hoursNum,
        notes: notes?.trim() || undefined,
      };

      const res = await fetch(
        `/api/staff/volunteers/${encodeURIComponent(String(volunteerId))}/hours`,
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          credentials: "include",
          body: JSON.stringify(payload),
        },
      );

      if (!res.ok) {
        let errMsg = `Request failed (${res.status})`;
        try {
          const body = await res.json();
          errMsg = body?.message || body?.error || errMsg;
        } catch {
          errMsg = res.statusText || errMsg;
        }
        alert(`Failed to log hours: ${errMsg}`);
        setSubmitting(false);
        return;
      }

      onSuccess?.();
      reset();
      onClose();
    } catch (error) {
      console.error(error);
      alert("Failed to log hours (network error).");
      setSubmitting(false);
    } finally {
      // if not already set false by earlier paths
      setSubmitting(false);
    }
  };

  return (
    <Dialog
      open={isOpen}
      onClose={handleClose}
      maxWidth="sm"
      fullWidth
      aria-labelledby="log-hours-dialog-title"
    >
      <form onSubmit={handleSubmit}>
        <DialogTitle id="log-hours-dialog-title">Log Hours</DialogTitle>

        <DialogContent dividers>
          <Box sx={{ mb: 2 }}>
            <Typography variant="body2" color="text.secondary">
              Record volunteer hours for this opportunity.
            </Typography>
          </Box>

          <Grid container spacing={2}>
            <Grid size={12}>
              <FormControl fullWidth size="small">
                <InputLabel id="opportunity-select-label">
                  Opportunity
                </InputLabel>
                <Select
                  labelId="opportunity-select-label"
                  id="opportunity-select"
                  value={opportunityId}
                  label="Opportunity"
                  onChange={handleSelectChange}
                  required
                >
                  <MenuItem value="">
                    <em>Select an opportunity...</em>
                  </MenuItem>
                  {opportunities.map((op) => (
                    <MenuItem key={String(op.id)} value={String(op.id)}>
                      {op.name}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>

            <Grid size={6}>
              <TextField
                id="hours-date"
                label="Date"
                type="date"
                value={date}
                onChange={(e) => setDate(e.target.value)}
                fullWidth
                size="small"
                InputLabelProps={{ shrink: true }}
                required
              />
            </Grid>

            <Grid size={6}>
              <TextField
                id="hours-input"
                label="Hours"
                type="number"
                value={hours}
                onChange={(e) => setHours(e.target.value)}
                inputProps={{ step: 0.25, min: 0.01 }}
                fullWidth
                size="small"
                required
              />
            </Grid>

            <Grid size={12}>
              <TextField
                id="hours-notes"
                label="Notes"
                value={notes}
                onChange={(e) => setNotes(e.target.value)}
                fullWidth
                size="small"
                multiline
                minRows={3}
              />
            </Grid>
          </Grid>
        </DialogContent>

        <DialogActions sx={{ px: 3, py: 2 }}>
          <Button onClick={handleClose} disabled={submitting}>
            Cancel
          </Button>

          <Button
            type="submit"
            variant="contained"
            disabled={submitting}
            startIcon={submitting ? <CircularProgress size={16} /> : undefined}
          >
            {submitting ? "Logging…" : "Log Hours"}
          </Button>
        </DialogActions>
      </form>
    </Dialog>
  );
};

export default LogHoursModal;
